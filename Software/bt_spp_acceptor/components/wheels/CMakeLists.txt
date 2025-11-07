# CMakeLists.txt for motor_control component
idf_component_register(
    SRCS "wheels.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver esp_timer # Uncomment and list dependencies if needed
)
