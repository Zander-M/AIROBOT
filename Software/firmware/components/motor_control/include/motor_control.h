#pragma once

// PINS
#define LED_BUILTIN 33
#define PIN_LEFT_FORWARD 26
#define PIN_LEFT_BACKWARD 27
#define PIN_RIGHT_FORWARD 18
#define PIN_RIGHT_BACKWARD 19

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 4095   // Full speed (2^12 - 1)

void setupPins();

typedef struct geometry_msgs__msg__Twist geometry_msgs__msg__Twist;
void setMotorFromTwist(geometry_msgs__msg__Twist* msg);

// Helper Macro function

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float fmap(float val, float in_min, float in_max, float out_min, float out_max);