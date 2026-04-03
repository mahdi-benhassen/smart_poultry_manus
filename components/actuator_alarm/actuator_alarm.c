#include "actuator_alarm.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ACT_ALARM";
static int s_gpio = -1;
static alarm_pattern_t s_pattern = ALARM_PATTERN_OFF;
static TaskHandle_t s_task_handle = NULL;
static volatile bool s_running = false;

static void alarm_task(void *arg)
{
    while (s_running) {
        switch (s_pattern) {
            case ALARM_PATTERN_CONTINUOUS:
                gpio_set_level(s_gpio, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case ALARM_PATTERN_INTERMITTENT:
                gpio_set_level(s_gpio, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(s_gpio, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case ALARM_PATTERN_SOS:
                // S: 3 short
                for (int i = 0; i < 3 && s_running; i++) {
                    gpio_set_level(s_gpio, 1);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(s_gpio, 0);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                vTaskDelay(pdMS_TO_TICKS(300));
                // O: 3 long
                for (int i = 0; i < 3 && s_running; i++) {
                    gpio_set_level(s_gpio, 1);
                    vTaskDelay(pdMS_TO_TICKS(450));
                    gpio_set_level(s_gpio, 0);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                vTaskDelay(pdMS_TO_TICKS(300));
                // S: 3 short
                for (int i = 0; i < 3 && s_running; i++) {
                    gpio_set_level(s_gpio, 1);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(s_gpio, 0);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case ALARM_PATTERN_OFF:
            default:
                gpio_set_level(s_gpio, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
        }
    }
    gpio_set_level(s_gpio, 0);
    vTaskDelete(NULL);
}

esp_err_t actuator_alarm_init(int gpio_num)
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
    s_pattern = ALARM_PATTERN_OFF;

    s_running = true;
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 5, &s_task_handle);

    ESP_LOGI(TAG, "Alarm initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_alarm_set_pattern(alarm_pattern_t pattern)
{
    if (s_gpio < 0) return ESP_ERR_INVALID_STATE;
    s_pattern = pattern;
    ESP_LOGI(TAG, "Alarm pattern set to %d", (int)pattern);
    return ESP_OK;
}

esp_err_t actuator_alarm_stop(void)
{
    s_pattern = ALARM_PATTERN_OFF;
    if (s_gpio >= 0) {
        gpio_set_level(s_gpio, 0);
    }
    return ESP_OK;
}

alarm_pattern_t actuator_alarm_get_pattern(void)
{
    return s_pattern;
}
