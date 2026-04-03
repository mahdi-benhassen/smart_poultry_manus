#include "actuator_cooler.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ACT_COOLER";
static int s_gpio = -1;
static bool s_state = false;

esp_err_t actuator_cooler_init(int gpio_num)
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
    s_state = false;
    ESP_LOGI(TAG, "Cooler initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_cooler_set(bool on)
{
    if (s_gpio < 0) return ESP_ERR_INVALID_STATE;
    gpio_set_level(s_gpio, on ? 1 : 0);
    s_state = on;
    ESP_LOGD(TAG, "Cooler %s", on ? "ON" : "OFF");
    return ESP_OK;
}

bool actuator_cooler_get(void)
{
    return s_state;
}
