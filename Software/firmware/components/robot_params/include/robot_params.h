#pragma once
// AIROBOT Hardware definitions

/********* HARDWARE DEFINITION *********/

// Robot geometry
#define WHEEL_RADIUS      0.042f   // meters
#define WHEEL_BASE        0.10f   // distance between wheels (m)
#define TICKS_PER_REV     2660.0f  // encoder CPR (counts per wheel revolution 1:40)
// #define TICKS_PER_REV     350.0f  // encoder CPR (counts per wheel revolution 1:300)
#define TWO_PI 6.28318530718f
#define PWM_MIN           120
#define PWM_MAX           255

// Wheel Direction

// Depending on RPM, the wheel direction might be different.
#define LEFT_DIR  -1
#define RIGHT_DIR 1

// PID coefficients
#define KP 1.
#define KI 0.
#define KD 0.

/********* PIN OUT *********/

// Battery Level
#define BATLVL 35

// Encoders
#define E1A 32
#define E1B 33
#define E2A 16 
#define E2B 17 

// Motors
#define ME1A 26 
#define ME1B 27 
#define ME2A 18
#define ME2B 19 

// LEDs
#define LED_PIN 4
#define LED_COUNT 1