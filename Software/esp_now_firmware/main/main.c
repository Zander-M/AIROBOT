/*
    ESP32 MicroROS diff-drive bot firmware
*/

#include <math.h>

#include "ros_node.h"
#include "motor_control.h"
#include "ota.h"
#include "led.h"
#include "battery.h"


#include <uros_network_interfaces.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#endif

static void battery_led_task(void *arg) {
    while (1) {
        if (low_battery) {
            for (int i = 0; i < 3; i++) {
                led_set_all(255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
                led_clear();
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

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

    // Battery
    battery_init();
    xTaskCreate(battery_read_task, "battery_update", 4096, NULL, 5, NULL);
    xTaskCreate(battery_led_task, "battery_led", 2048, NULL, 4, NULL);

    // ROS task
    // Run ROS in FreeRTOS stack
    xTaskCreate(ros_task, "ros_task", 8192, NULL, 5, NULL);

}