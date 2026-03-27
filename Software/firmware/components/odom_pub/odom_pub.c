/*
    Odom Publish
*/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>

#include "odom_pub.h"
#include "robot_params.h"
#include "wheels.h"

static const char *TAG = "odom_pub";

typedef struct {
    float x;
    float y;
    float theta;
    float linear_x;
    float angular_z;
    int64_t prev_left_count;
    int64_t prev_right_count;
    int64_t prev_time_us;
    builtin_interfaces__msg__Time stamp;
    bool initialized;
} odom_state_t;

static odom_state_t odom_state;

static inline float normalize_angle(float angle)
{
    const float pi = 0.5f * TWO_PI;

    while (angle > pi) {
        angle -= TWO_PI;
    }
    while (angle < -pi) {
        angle += TWO_PI;
    }
    return angle;
}

static inline float ticks_to_distance(int64_t ticks)
{
    return ((float)ticks * TWO_PI * WHEEL_RADIUS) / TICKS_PER_REV;
}

static void fill_timestamp(builtin_interfaces__msg__Time *stamp)
{
    int64_t time_ns = 0;

    if (rmw_uros_epoch_synchronized()) {
        time_ns = rmw_uros_epoch_nanos();
    } else {
        time_ns = esp_timer_get_time() * 1000LL;
    }

    stamp->sec = (int32_t)(time_ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(time_ns % 1000000000LL);
}

static void fill_orientation(geometry_msgs__msg__Quaternion *orientation)
{
    orientation->x = 0.0;
    orientation->y = 0.0;
    orientation->z = sinf(odom_state.theta * 0.5f);
    orientation->w = cosf(odom_state.theta * 0.5f);
}

void odom_init(void)
{
    memset(&odom_state, 0, sizeof(odom_state));
}

bool odom_prepare_msg(nav_msgs__msg__Odometry *msg)
{
    if (!nav_msgs__msg__Odometry__init(msg)) {
        ESP_LOGE(TAG, "Failed to initialize odometry message");
        return false;
    }

    if (!rosidl_runtime_c__String__assign(&msg->header.frame_id, "odom")) {
        ESP_LOGE(TAG, "Failed to assign odom frame");
        nav_msgs__msg__Odometry__fini(msg);
        return false;
    }

    if (!rosidl_runtime_c__String__assign(&msg->child_frame_id, "base_link")) {
        ESP_LOGE(TAG, "Failed to assign base frame");
        nav_msgs__msg__Odometry__fini(msg);
        return false;
    }

    for (size_t i = 0; i < 36; ++i) {
        msg->pose.covariance[i] = 0.0;
        msg->twist.covariance[i] = 0.0;
    }

    msg->pose.covariance[0] = 0.02;
    msg->pose.covariance[7] = 0.02;
    msg->pose.covariance[14] = 1000000.0;
    msg->pose.covariance[21] = 1000000.0;
    msg->pose.covariance[28] = 1000000.0;
    msg->pose.covariance[35] = 0.05;

    msg->twist.covariance[0] = 0.02;
    msg->twist.covariance[7] = 0.02;
    msg->twist.covariance[14] = 1000000.0;
    msg->twist.covariance[21] = 1000000.0;
    msg->twist.covariance[28] = 1000000.0;
    msg->twist.covariance[35] = 0.05;

    return true;
}

void odom_update(void)
{
    int64_t left_count = 0;
    int64_t right_count = 0;
    const int64_t now_us = esp_timer_get_time();

    wheel_get_counts(&left_count, &right_count);

    if (!odom_state.initialized) {
        odom_state.prev_left_count = left_count;
        odom_state.prev_right_count = right_count;
        odom_state.prev_time_us = now_us;
        odom_state.initialized = true;
    } else {
        const int64_t raw_left_delta = left_count - odom_state.prev_left_count;
        const int64_t raw_right_delta = right_count - odom_state.prev_right_count;
        const int64_t dt_us = now_us - odom_state.prev_time_us;

        if (dt_us > 0) {
            const float dt = (float)dt_us / 1000000.0f;
            const int64_t left_delta_ticks = raw_left_delta * LEFT_DIR;
            const int64_t right_delta_ticks = raw_right_delta * RIGHT_DIR;
            const float left_distance = ticks_to_distance(left_delta_ticks);
            const float right_distance = ticks_to_distance(right_delta_ticks);
            const float delta_s = 0.5f * (left_distance + right_distance);
            const float delta_theta = (right_distance - left_distance) / WHEEL_BASE;
            const float heading_mid = odom_state.theta + (0.5f * delta_theta);

            odom_state.x += delta_s * cosf(heading_mid);
            odom_state.y += delta_s * sinf(heading_mid);
            odom_state.theta = normalize_angle(odom_state.theta + delta_theta);
            odom_state.linear_x = delta_s / dt;
            odom_state.angular_z = delta_theta / dt;
        }

        odom_state.prev_left_count = left_count;
        odom_state.prev_right_count = right_count;
        odom_state.prev_time_us = now_us;
    }

    fill_timestamp(&odom_state.stamp);
}

void odom_fill_msg(nav_msgs__msg__Odometry *msg)
{
    msg->header.stamp = odom_state.stamp;
    msg->pose.pose.position.x = odom_state.x;
    msg->pose.pose.position.y = odom_state.y;
    msg->pose.pose.position.z = 0.0;
    fill_orientation(&msg->pose.pose.orientation);

    msg->twist.twist.linear.x = odom_state.linear_x;
    msg->twist.twist.linear.y = 0.0;
    msg->twist.twist.linear.z = 0.0;
    msg->twist.twist.angular.x = 0.0;
    msg->twist.twist.angular.y = 0.0;
    msg->twist.twist.angular.z = odom_state.angular_z;
}
