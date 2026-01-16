# CMakeLists.txt for motor_control component

idf_component_register(
    SRCS "motor_control.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver esp_timer microros_prebuilt wheels robot_params# Uncomment and list dependencies if needed
)
