#include "sensor_door.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "DOOR_SENSOR";
static gpio_num_t door_gpio_pin;

// Debounce parameters
#define DEBOUNCE_TIME_MS 50
static int64_t last_debounce_time = 0;
static bool last_door_state = false;

// Moving average filter parameters (for state, can be used for more complex logic)
#define MA_WINDOW_SIZE 10
static bool door_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

esp_err_t sensor_door_init(gpio_num_t door_pin)
{
    door_gpio_pin = door_pin;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << door_gpio_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Assuming normally closed switch, pull-up to detect open
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Door sensor GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Door sensor initialized on GPIO %d", door_gpio_pin);
    return ESP_OK;
}

esp_err_t sensor_door_read(bool *door_open)
{
    if (door_open == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool current_gpio_state = gpio_get_level(door_gpio_pin);
    bool debounced_state = current_gpio_state; // Start with current state

    int64_t now = esp_timer_get_time() / 1000; // current time in ms

    // Simple debounce logic
    if ((now - last_debounce_time) > DEBOUNCE_TIME_MS) {
        if (current_gpio_state != last_door_state) {
            last_debounce_time = now;
            last_door_state = current_gpio_state;
        }
    }
    debounced_state = last_door_state;

    // Assuming HIGH means door is open (e.g., switch opens circuit, pull-up makes it high)
    bool current_door_state = debounced_state; 

    // Apply moving average filter (for state, can be used for more complex logic)
    door_readings[ma_index] = current_door_state;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    int open_count = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        if (door_readings[i]) {
            open_count++;
        }
    }

    // If more than half of the recent readings indicate open, consider it open
    *door_open = (open_count > (count / 2));

    ESP_LOGD(TAG, "Door Raw GPIO: %d, Debounced: %d, Filtered State: %s",
             current_gpio_state, debounced_state, *door_open ? "OPEN" : "CLOSED");

    return ESP_OK;
}
