/*
    Motor driver
*/
#include <stdio.h>
#include <math.h>

#include <geometry_msgs/msg/twist.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "wheels.h"
#include "robot_params.h"


static inline float radps_to_ticks(float radps) {
    return radps * (TICKS_PER_REV / TWO_PI);
}

void motor_init(void){
    wheel_init();
    PIDConfig cfg = {(float) KP, 
                     (float) KI, 
                     (float) KD
                    };
    wheel_setPID(cfg);
}

void setMotorFromTwist(geometry_msgs__msg__Twist* msg) {
    // obtain Twist
    float v = msg->linear.x;
    float w = msg->angular.z;

    // Convert to angular velocity (rad/s)
    float wl = (v-w*(WHEEL_BASE/ 2.0f)) / WHEEL_RADIUS;
    float wr = (v+w*(WHEEL_BASE/ 2.0f)) / WHEEL_RADIUS;

    float l_tps = radps_to_ticks(wl);
    float r_tps = radps_to_ticks(wr);

    setLeftTarget(l_tps);
    setRightTarget(r_tps);

    // printf("Twist: v=%.2f m/s, w=%.2f rad/s | Targets: L=%.2f tps, R=%.2f tps\n",
    //        v, w, l_tps, r_tps);
}

void motor_update_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    TickType_t last = xTaskGetTickCount();

    while (1) {
        wheel_run();  // PID + PWM update
        vTaskDelayUntil(&last, period);
    }
}
