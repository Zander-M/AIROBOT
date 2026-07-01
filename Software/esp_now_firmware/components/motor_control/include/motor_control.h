#pragma once


void motor_init();

typedef struct geometry_msgs__msg__Twist geometry_msgs__msg__Twist;
void setMotorFromTwist(geometry_msgs__msg__Twist* msg);
void motor_update_task(void *arg); // Callback 

// Helper Macro function

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float fmap(float val, float in_min, float in_max, float out_min, float out_max);