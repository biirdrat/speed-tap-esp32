#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "i2c_lcd.h"

#define LCD_MESSAGE_BUFFER_SIZE 17
#define NUM_BUTTONS 4

typedef enum 
{
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED = 1
} button_state;

typedef struct 
{
    uint8_t button_pin;
    uint8_t current_input;
    uint8_t previous_input;
    button_state state;
} button;

static const char *TAG = "MAIN";

const uint8_t LED_PIN = 2;
const uint8_t LED0_PIN = 4;
const uint8_t LED1_PIN = 16;
const uint8_t LED2_PIN = 17;
const uint8_t LED3_PIN = 5;
const uint8_t BUTTON0_PIN = 27;
const uint8_t BUTTON1_PIN = 26;
const uint8_t BUTTON2_PIN = 5;
const uint8_t BUTTON3_PIN = 33;

SemaphoreHandle_t process_buttons_semaphore;

char lcd_message_buffer[LCD_MESSAGE_BUFFER_SIZE];

gptimer_handle_t program_timer = NULL;
gptimer_handle_t process_buttons_timer = NULL;
uint64_t start_count = 0;
uint64_t end_count = 0;

button buttons_array[NUM_BUTTONS];

void process_buttons_task(void *pvParameter);
void initialize_gpio_pins();
void initialize_buttons();
void initialize_lcd();
void initialize_timers();
void initialize_semaphores();

void app_main(void)
{
    // Print a starting message
    ESP_LOGI(TAG, "Main Program Running!\n");

    initialize_gpio_pins();

    initialize_buttons();

    initialize_lcd();

    initialize_timers();

    initialize_semaphores();

    ESP_LOGI(TAG, "Main Program Finished!\n");

    xTaskCreate(&process_buttons_task, "Process Buttons Task", 2048, NULL, 5, NULL);
}

static bool IRAM_ATTR process_buttons_timer_on_alarm(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_data
    )
{
    BaseType_t high_task_awoken = pdFALSE;

    // Give the semaphore from ISR
    xSemaphoreGiveFromISR(process_buttons_semaphore, &high_task_awoken);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void process_buttons_task(void *pvParameter)
{
    while (1)
    {
        // Wait indefinitely for the semaphore
        if (xSemaphoreTake(process_buttons_semaphore, portMAX_DELAY) == pdTRUE)
        {
            int button_state = gpio_get_level(BUTTON0_PIN);
            ESP_LOGI(TAG, "GPIO 27 state: %d", button_state);
        }
    }
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

void initialize_buttons()
{
    buttons_array[0].button_pin = BUTTON0_PIN;
    buttons_array[1].button_pin = BUTTON1_PIN;
    buttons_array[2].button_pin = BUTTON2_PIN;
    buttons_array[3].button_pin = BUTTON3_PIN;

    for(int button_idx = 0; button_idx < NUM_BUTTONS; button_idx++)
    {
        buttons_array[button_idx].current_input = 0;
        buttons_array[button_idx].previous_input = 0;
        buttons_array[button_idx].state = BUTTON_RELEASED;
    }
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
        lcd_put_cur(0, 0);
    
        // Send a message to the LCD
        snprintf(lcd_message_buffer, LCD_MESSAGE_BUFFER_SIZE, "hello");
        lcd_send_string(lcd_message_buffer);
}

void initialize_timers()
{
    
    // Set the configuration for the program timer
    gptimer_config_t program_timer_config=
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    // Set the configuration for the detect button timer
    gptimer_config_t process_buttons_timer_config = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    // Set the callback function for the detect button timer
    gptimer_event_callbacks_t process_buttons_callbacks = 
    {
        .on_alarm = process_buttons_timer_on_alarm,
    };

    // Set the configuration for the detect button timer
    gptimer_alarm_config_t process_buttons_alarm_config = 
    {
        .alarm_count = 2000000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    // Create the program timer and enable timer
    ESP_ERROR_CHECK(gptimer_new_timer(&program_timer_config, &program_timer));
    ESP_ERROR_CHECK(gptimer_enable(program_timer));
    ESP_ERROR_CHECK(gptimer_start(program_timer));

    // Create the detect button timer, register callbacks, and enable timer
    ESP_ERROR_CHECK(gptimer_new_timer(&process_buttons_timer_config, &process_buttons_timer));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(process_buttons_timer, &process_buttons_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(process_buttons_timer, &process_buttons_alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(process_buttons_timer));
    ESP_ERROR_CHECK(gptimer_start(process_buttons_timer));
}

void initialize_semaphores()
{
    process_buttons_semaphore = xSemaphoreCreateBinary();
    if(process_buttons_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create process_buttons_semaphore\n");
    }
}