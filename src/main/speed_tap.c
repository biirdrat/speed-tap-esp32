#include <stdio.h>
#include "esp_random.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "i2c_lcd.h"

#define LCD_MESSAGE_BUFFER_SIZE 17
#define NUM_LEDS 4
#define NUM_BUTTONS 6
#define NUM_ACTION_BUTTONS 4

typedef enum
{
    IDLE = 0,
    INTERMISSION = 1,
    GAME_IN_PROGRESS = 2,
    TIMEOUT = 3,
    GAME_OVER = 4
} game_state;

typedef enum 
{
    BUTTON_WAS_RELEASED = 0,
    BUTTON_WAS_PRESSED = 1,
    BUTTON_WAITING_RESET = 2
} button_state;

typedef struct 
{
    uint8_t button_pin;
    uint8_t current_input;
    uint8_t previous_input;
    button_state state;
} button;

static const char *TAG = "MAIN";

const uint32_t TIMER_TICKS_PER_SECOND = 1000000;
const uint8_t INTERMISSION_TIME_S = 3;
const uint8_t TOP_ROW = 0;
const uint8_t BOTTOM_ROW = 1;
const uint8_t BUTTON_NOT_PRESSED = 0;
const uint8_t BUTTON_PRESSED = 1;
const uint8_t LED_PIN = 2;
const uint8_t LED0_PIN = 4;
const uint8_t LED1_PIN = 16;
const uint8_t LED2_PIN = 17;
const uint8_t LED3_PIN = 5;
const uint8_t START_BUTTON_PIN = 13;
const uint8_t STOP_BUTTON_PIN = 12;
const uint8_t ACTION_BUTTON0_PIN = 27;
const uint8_t ACTION_BUTTON1_PIN = 26;
const uint8_t ACTION_BUTTON2_PIN = 25;
const uint8_t ACTION_BUTTON3_PIN = 33;

SemaphoreHandle_t process_buttons_semaphore;
SemaphoreHandle_t intermission_semaphore;
SemaphoreHandle_t button_data_mutex;
SemaphoreHandle_t game_state_mutex;

gptimer_handle_t program_timer = NULL;
gptimer_handle_t process_buttons_timer = NULL;
uint64_t start_count = 0;
uint64_t end_count = 0;

uint8_t leds_array[NUM_LEDS] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN};
button buttons_array[NUM_BUTTONS];
game_state current_game_state = IDLE;
uint8_t intermission_count = 0;
uint8_t correct_button = 255;
uint16_t current_score = 0;

void game_control_task(void *pvParameter);
void process_buttons_task(void *pvParameter);
void intermission_task(void *pvParameter);
void initialize_leds();
void initialize_buttons();
void initialize_lcd();
void initialize_timers();
void initialize_freertos_objects();
void turn_off_all_leds();
void generate_new_target();
void update_lcd_score();
void start_game();

void app_main(void)
{
    // Print a starting message
    ESP_LOGI(TAG, "Main Program Running!\n");
    
    initialize_leds();

    initialize_buttons();

    initialize_lcd();

    initialize_timers();

    initialize_freertos_objects();

    ESP_LOGI(TAG, "Main Program Finished!\n");

    xTaskCreate(&process_buttons_task, "Process Buttons Task", 2048, NULL, 24, NULL);
    xTaskCreate(&game_control_task, "Game Control Task", 2048, NULL, 23, NULL);
    xTaskCreate(&intermission_task, "Intermission Task", 2048, NULL, 22, NULL);
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

void game_control_task(void *pvParameter)
{
    while (1)
    {
        if(xSemaphoreTake(button_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            if(xSemaphoreTake(game_state_mutex, portMAX_DELAY) == pdTRUE)
            {
                switch(current_game_state)
                {   
                    // Waiting for the start button to be pressed
                    case IDLE:
                        if(buttons_array[0].state == BUTTON_WAS_PRESSED)
                        {
                            intermission_count = INTERMISSION_TIME_S;
                            current_game_state = INTERMISSION;
                            xSemaphoreGive(intermission_semaphore);
                        }
                        break;
                    
                    // Game starting
                    case INTERMISSION:
                        break;

                    // Game is in progress
                    case GAME_IN_PROGRESS:
                        // Check if stop button was pressed
                        if(buttons_array[1].state == BUTTON_WAS_PRESSED)
                        {
                            buttons_array[1].state = BUTTON_WAITING_RESET;
                            current_game_state = IDLE;
                        }
                        else
                        {
                            uint8_t num_buttons_pressed = 0;
                            bool correct_button_pressed = false;
                            for(int button_idx = NUM_BUTTONS - NUM_ACTION_BUTTONS; button_idx < NUM_BUTTONS; button_idx++)
                            {
                                // Check if the button was pressed
                                if(buttons_array[button_idx].state == BUTTON_WAS_PRESSED)
                                {
                                    // Increment the number of buttons pressed
                                    num_buttons_pressed++;
                                    
                                    // Check if the button pressed is the correct button
                                    if(button_idx == correct_button)
                                    {
                                        correct_button_pressed = true;
                                        ESP_LOGI(TAG, "Correct button %d was pressed\n", button_idx);
                                    }

                                    // Make sure the button is released by user before next check
                                    buttons_array[button_idx].state = BUTTON_WAITING_RESET;
                                }
                            }
                            
                            // User did not make an input
                            if(num_buttons_pressed == 0)
                            {
                                break;
                            }
                            // User selected single correct button
                            else if(num_buttons_pressed == 1 && correct_button_pressed)
                            {
                                current_score++;
                                update_lcd_score();
                                generate_new_target();
                            }
                            // User failed to select correct button
                            else
                            {
                                turn_off_all_leds();
                                current_game_state = TIMEOUT;
                            }
                        }
                        break;
                    
                    // User pressed wrong button
                    case TIMEOUT:
                        break;
                    
                    // Game is finished
                    case GAME_OVER:
                        break;

                    default:
                        break;
                }
                xSemaphoreGive(game_state_mutex);
                xSemaphoreGive(button_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void process_buttons_task(void *pvParameter)
{
    while (1)
    {
        // Wait indefinitely for the semaphore
        if (xSemaphoreTake(process_buttons_semaphore, portMAX_DELAY) == pdTRUE)
        {
            if(xSemaphoreTake(button_data_mutex, portMAX_DELAY) == pdTRUE)
            {
                for(int button_idx = 0; button_idx < NUM_BUTTONS; button_idx++)
                {
                    buttons_array[button_idx].current_input = 
                        gpio_get_level(buttons_array[button_idx].button_pin);

                    int current_input = buttons_array[button_idx].current_input;
                    int previous_input = buttons_array[button_idx].previous_input;
                    
                    if(previous_input == BUTTON_NOT_PRESSED && current_input == BUTTON_PRESSED)
                    {
                        ESP_LOGI(TAG, "Button %d was pressed\n", button_idx);
                        buttons_array[button_idx].state = BUTTON_WAS_PRESSED;
                    }
                    else if(previous_input == BUTTON_PRESSED && current_input == BUTTON_NOT_PRESSED)
                    {
                        buttons_array[button_idx].state = BUTTON_WAS_RELEASED;
                    }
                    buttons_array[button_idx].previous_input = current_input;
                }
                xSemaphoreGive(button_data_mutex);
            }
        }
    }
}

void intermission_task(void *pvParameter)
{
    while (1)
    {
        if(xSemaphoreTake(intermission_semaphore, portMAX_DELAY) == pdTRUE)
        {
            lcd_write_string(BOTTOM_ROW, "Game Starting: %i", (int)(intermission_count));
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Decrement intermission count until it reaches 0
            if(intermission_count != 0)
            {
                intermission_count--;
                xSemaphoreGive(intermission_semaphore);
            }
            // Exit intermission and start the game
            else
            {
                start_game();
                if(xSemaphoreTake(game_state_mutex, portMAX_DELAY) == pdTRUE)
                {
                    current_game_state = GAME_IN_PROGRESS;
                    xSemaphoreGive(game_state_mutex);
                }
            }
        }
    }
}

void initialize_leds()
{
    // Configure the LEDS GPIO as an output
    gpio_config_t io_config_output = 
    {
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

    for(int led_idx = 0; led_idx < NUM_LEDS; led_idx++)
    {
        gpio_set_level(leds_array[led_idx], 0);
    }
}

void initialize_buttons()
{
    // Configure buttons as inputs
    gpio_config_t io_config_input = 
    {
        .pin_bit_mask = (1ULL << START_BUTTON_PIN) |
                        (1ULL << STOP_BUTTON_PIN) |
                        (1ULL << ACTION_BUTTON0_PIN) |
                        (1ULL << ACTION_BUTTON1_PIN) |
                        (1ULL << ACTION_BUTTON2_PIN) |
                        (1ULL << ACTION_BUTTON3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_config_input);

    buttons_array[0].button_pin = START_BUTTON_PIN;
    buttons_array[1].button_pin = STOP_BUTTON_PIN;
    buttons_array[2].button_pin = ACTION_BUTTON0_PIN;
    buttons_array[3].button_pin = ACTION_BUTTON1_PIN;
    buttons_array[4].button_pin = ACTION_BUTTON2_PIN;
    buttons_array[5].button_pin = ACTION_BUTTON3_PIN;

    for(int button_idx = 0; button_idx < NUM_BUTTONS; button_idx++)
    {
        buttons_array[button_idx].current_input = 0;
        buttons_array[button_idx].previous_input = 0;
        buttons_array[button_idx].state = BUTTON_WAS_RELEASED;
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
    
        lcd_write_string(TOP_ROW, "Speed Tap Game");
}

void initialize_timers()
{
    
    // Set the configuration for the program timer
    gptimer_config_t program_timer_config=
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_TICKS_PER_SECOND,
    };

    // Set the configuration for the detect button timer
    gptimer_config_t process_buttons_timer_config = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_TICKS_PER_SECOND,
    };

    // Set the callback function for the detect button timer
    gptimer_event_callbacks_t process_buttons_callbacks = 
    {
        .on_alarm = process_buttons_timer_on_alarm,
    };

    // Set the configuration for the detect button timer
    gptimer_alarm_config_t process_buttons_alarm_config = 
    {
        .alarm_count = (uint64_t)(0.05 * TIMER_TICKS_PER_SECOND),
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

void initialize_freertos_objects()
{
    process_buttons_semaphore = xSemaphoreCreateBinary();
    if(process_buttons_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create process_buttons_semaphore\n");
    }

    intermission_semaphore = xSemaphoreCreateBinary();
    if(intermission_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create intermission_semaphore\n");
    }

    button_data_mutex = xSemaphoreCreateMutex();
    if(button_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create button_data_mutex\n");
    }

    game_state_mutex = xSemaphoreCreateMutex();
    if(game_state_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create game_state_mutex\n");
    }
}

void turn_off_all_leds()
{
    for(int led_idx = 0; led_idx < NUM_LEDS; led_idx++)
    {
        gpio_set_level(leds_array[led_idx], 0);
    }
}

void generate_new_target()
{   
    uint8_t last_correct_button = correct_button;
    uint8_t new_correct_button = 0;
    do 
    {
        // Generate a random number between 2 and 5 (inclusive)
        new_correct_button = (esp_random() % NUM_ACTION_BUTTONS) + 2;

    // Make sure the new button is different from the last one
    } while (new_correct_button == last_correct_button);

    // Update the correct button
    correct_button = new_correct_button;

    // Turn on the led for the new correct button
    for(int led_idx = 0; led_idx < NUM_LEDS; led_idx++)
    {
        if( led_idx == (new_correct_button - 2))
        {
            gpio_set_level(leds_array[led_idx], 1);
        }
        else
        {
            gpio_set_level(leds_array[led_idx], 0);
        }
    }
}

void update_lcd_score()
{
    lcd_write_string(BOTTOM_ROW, "Score: %i", (int)(current_score)); 
}

void start_game()
{
    lcd_write_string(TOP_ROW, "Game Started!");
    current_score = 0;
    update_lcd_score();
    generate_new_target();
}