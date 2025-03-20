#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "i2c_lcd.h"

#define LCD_MESSAGE_BUFFER_SIZE 17

uint8_t const LED_PIN = 2;
uint8_t LED0_PIN = 4;
uint8_t LED1_PIN = 16;
uint8_t LED2_PIN = 17;
uint8_t LED3_PIN = 5;
uint8_t BUTTON0_PIN = 27;
uint8_t BUTTON1_PIN = 26;
uint8_t BUTTON2_PIN = 5;
uint8_t BUTTON3_PIN = 33;

static const char *TAG = "MAIN";

char lcd_message_buffer[LCD_MESSAGE_BUFFER_SIZE];

gptimer_handle_t detect_button_timer = NULL;

void initialize_gpio_pins();
void initialize_lcd();
void initialize_timers();


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
    // Print a starting message
    ESP_LOGI(TAG, "Main Program Running!\n");

    initialize_gpio_pins();

    initialize_lcd();
    // initialize_timers();

    ESP_LOGI(TAG, "Main Program Finished!\n");
    // Create the FreeRTOS task to blink LED
    // xTaskCreate(&led_blink_task, "LED Blink Task", 2048, NULL, 5, NULL);
}

static bool IRAM_ATTR detect_button_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    ESP_EARLY_LOGI(TAG, "Timer Alarm Triggered! Count: %llu", edata->count_value);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void initialize_gpio_pins()
{
    // Configure buttons as inputs
    gpio_config_t io_config_input = {
        .pin_bit_mask = (1ULL << BUTTON0_PIN) |
                        (1ULL << BUTTON1_PIN) |
                        (1ULL << BUTTON2_PIN) |
                        (1ULL << BUTTON3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_config_input);

    // Configure the LED GPIO as an output
    gpio_config_t io_config_output = {
        .pin_bit_mask = (1ULL << LED0_PIN) |
                        (1ULL << LED1_PIN) |
                        (1ULL << LED2_PIN) |
                        (1ULL << LED3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_config_output);
}

void initialize_lcd()
{
        // Initialize the I2C bus
        ESP_ERROR_CHECK(i2c_master_init());
    
        // Initialize the LCD
        lcd_init();
    
        // Clear the LCD screen
        lcd_clear();
    
        // Put the cursor at the first position
        lcd_put_cur(1, 0);
    
        // Send a message to the LCD
        snprintf(lcd_message_buffer, LCD_MESSAGE_BUFFER_SIZE, "hello");
        lcd_send_string(lcd_message_buffer);s
}

void initialize_timers()
{
    // Set the configuration for the detect button timer
    gptimer_config_t detect_button_timer_config = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    // Set the callback function for the detect button timer
    gptimer_event_callbacks_t detect_button_callbacks = 
    {
        .on_alarm = detect_button_timer_on_alarm,
    };

    // Set the configuration for the detect button timer
    gptimer_alarm_config_t detect_button_alarm_config = 
    {
        .alarm_count = 2000000, // 1-second interval
        .reload_count = 0,      // Restart count from zero after alarm
        .flags.auto_reload_on_alarm = true, // Keeps triggering every second
    };

    // Create the detect button timer, register callbacks, and enable timer
    ESP_ERROR_CHECK(gptimer_new_timer(&detect_button_timer_config, &detect_button_timer));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(detect_button_timer, &detect_button_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(detect_button_timer, &detect_button_alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(detect_button_timer));
    ESP_ERROR_CHECK(gptimer_start(detect_button_timer));
}

