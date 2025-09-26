/*
    ESP32 MicroROS diff-drive bot firmware
*/

#include "ros_node.h"
#include "motor_control.h"
#include "ota.h"

#include <uros_network_interfaces.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#endif

void app_main(void){
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif 

    setupMotor();
    // ota_init();

    // Run ROS in FreeRTOS stack
    xTaskCreate(ros_task, "ros_task", 8192, NULL, 5, NULL);
}