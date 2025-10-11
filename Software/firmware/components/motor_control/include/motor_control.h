#pragma once

// Robot geometry
#define WHEEL_RADIUS      0.042f   // meters
#define WHEEL_BASE        0.10f   // distance between wheels (m)
#define TICKS_PER_REV     280.0f  // encoder CPR (counts per wheel revolution)

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 4095   // Full speed (2^12 - 1)

// PID coefficients
#define KP 0.6
#define KI 0.8
#define KD 0

void setupMotor();

typedef struct geometry_msgs__msg__Twist geometry_msgs__msg__Twist;
void setMotorFromTwist(geometry_msgs__msg__Twist* msg);

void motor_update_task(void *arg); //Callback 

// Helper Macro function

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float fmap(float val, float in_min, float in_max, float out_min, float out_max);