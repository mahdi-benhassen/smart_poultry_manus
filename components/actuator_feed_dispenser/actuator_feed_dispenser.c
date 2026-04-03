#include "actuator_feed_dispenser.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ACT_FEED";
static int s_gpio = -1;
static bool s_active = false;
static esp_timer_handle_t s_timer = NULL;

static void feed_timer_callback(void *arg)
{
    if (s_gpio >= 0) {
        gpio_set_level(s_gpio, 0);
    }
    s_active = false;
    ESP_LOGI(TAG, "Feed dispensing complete");
}

esp_err_t actuator_feed_dispenser_init(int gpio_num)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(gpio_num, 0);
    s_gpio = gpio_num;

    esp_timer_create_args_t timer_args = {
        .callback = feed_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "feed_timer",
    };
    err = esp_timer_create(&timer_args, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Timer create failed: %s", esp_err_to_name(err));
        return err;
    }

    s_active = false;
    ESP_LOGI(TAG, "Feed dispenser initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_feed_dispenser_dispense(uint32_t duration_ms)
{
    if (s_gpio < 0 || s_timer == NULL) return ESP_ERR_INVALID_STATE;
    if (s_active) {
        ESP_LOGW(TAG, "Feed dispenser already active");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(s_gpio, 1);
    s_active = true;

    esp_err_t err = esp_timer_start_once(s_timer, (uint64_t)duration_ms * 1000);
    if (err != ESP_OK) {
        gpio_set_level(s_gpio, 0);
        s_active = false;
        ESP_LOGE(TAG, "Timer start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Dispensing feed for %lu ms", (unsigned long)duration_ms);
    return ESP_OK;
}

bool actuator_feed_dispenser_is_active(void)
{
    return s_active;
}
