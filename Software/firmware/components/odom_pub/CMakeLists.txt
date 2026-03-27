# CMakeLists.txt for odom_pub component

idf_component_register(
    SRCS "odom_pub.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver esp_timer microros_prebuilt wheels robot_params # Uncomment and list dependencies if needed
)
