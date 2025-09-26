# CMakeLists.txt for motor_control component

idf_component_register(
    SRCS "wheels.c" "motor_control.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver esp_timer microros_prebuilt# Uncomment and list dependencies if needed
)
