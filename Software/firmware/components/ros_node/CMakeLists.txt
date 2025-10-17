idf_component_register (
    SRCS "ros_node.c"
    INCLUDE_DIRS "include"
    REQUIRES motor_control led microros_prebuilt
)