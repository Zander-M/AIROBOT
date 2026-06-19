idf_component_register(
    SRCS "battery.c" # Add other source files here
    INCLUDE_DIRS "include" # Specify include directories
    REQUIRES driver esp_timer esp_adc microros_prebuilt robot_params
)


