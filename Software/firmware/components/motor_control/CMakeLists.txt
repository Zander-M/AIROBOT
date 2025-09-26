# CMakeLists.txt for motor_control component

idf_component_register(
    SRCS "motor_control.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver microros_prebuilt# Uncomment and list dependencies if needed
)
