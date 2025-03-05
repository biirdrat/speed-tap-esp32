#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c_lcd.h"

#define LED_PIN GPIO_NUM_2  // Change this to your LED pin number

static const char *TAG = "MAIN";

void led_blink_task(void *pvParameter)
{
    // Configure the LED GPIO as an output
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(LED_PIN, 1); // Turn LED ON
        printf("LED ON\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

        gpio_set_level(LED_PIN, 0); // Turn LED OFF
        printf("LED OFF\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Main Program Running!\n");

    // Create the FreeRTOS task to blink LED
    // xTaskCreate(&led_blink_task, "LED Blink Task", 2048, NULL, 5, NULL);
    
}