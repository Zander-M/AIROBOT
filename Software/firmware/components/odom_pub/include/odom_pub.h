#pragma once

#include <stdbool.h>

typedef struct nav_msgs__msg__Odometry nav_msgs__msg__Odometry;

void odom_init(void);
bool odom_prepare_msg(nav_msgs__msg__Odometry *msg);
void odom_update(void);
void odom_fill_msg(nav_msgs__msg__Odometry *msg);
