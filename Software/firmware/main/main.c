/*
    ESP32 MicroROS diff-drive bot firmware
*/

#include <math.h>

#include "ros_node.h"
#include "motor_control.h"
#include "ota.h"
#include "led.h"

#define LED_PIN 4
#define LED_COUNT 2

#include <uros_network_interfaces.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#endif

void app_main(void){

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif 

    // LED
    led_init();
    led_clear();

    // Motor
    motor_init();
    xTaskCreate(motor_update_task, "motor_update", 4096, NULL, 5, NULL);

    // OTA 
    // ota_init();

    // ROS task
    // Run ROS in FreeRTOS stack
    xTaskCreate(ros_task, "ros_task", 8192, NULL, 5, NULL);

}