/*
    Micro-ROS related code
*/

#include <stdio.h>

#include "ros_node.h"
#include "motor_control.h"
#include "led.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/color_rgba.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	#include <rmw_microros/rmw_microros.h>
#endif

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#endif

// Callback declaration

void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void led_callback(const void *msgin);

geometry_msgs__msg__Twist vel_msg;
std_msgs__msg__ColorRGBA led_msg;

void ros_task(void *arg) {
    // Init micro-ROS support
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

	#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
		rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
		// Static Agent IP and port can be used instead of autodisvery.
		RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
	#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create ros node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ros_esp32_diffdrive", "", &support));

    // Create subscriber
    rcl_subscription_t vel_sub;
    RCCHECK(rclc_subscription_init_default(
        &vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    rcl_subscription_t led_sub;
    RCCHECK(rclc_subscription_init_default(
        &led_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "/led_color"));

    // Timer
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));

    // Executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &vel_sub, &vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//Callbacks
void cmd_vel_callback(const void * msgin){
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    printf("Received /cmd_vel: linear=%.2f angular=%.2f\n",
           msg->linear.x, msg->angular.z);
    vel_msg = *msg;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    setMotorFromTwist(&vel_msg);
}

void led_callback(const void *msgin){
    const std_msgs__msg__ColorRGBA *msg = (const std_msgs__msg__ColorRGBA *)msgin;

    uint8_t r = (uint8_t)(msg->r * 255.0f);
    uint8_t g = (uint8_t)(msg->g * 255.0f);
    uint8_t b = (uint8_t)(msg->b * 255.0f);

    ESP_LOGI("ros_node", "Received /led_color: R=%d G=%d B=%d", r, g, b);
    led_set_pixel(0, r, g, b);
    led_refresh();
}