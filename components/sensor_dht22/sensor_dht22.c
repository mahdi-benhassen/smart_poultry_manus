#include "sensor_dht22.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "DHT22_SENSOR";
static gpio_num_t dht_gpio_pin;

#define DHT_MAX_CYCLES 1000
#define DHT_DATA_BITS 40

#define MA_WINDOW_SIZE 10
static float temp_readings[MA_WINDOW_SIZE];
static float hum_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

static void dht_gpio_set_input(void)
{
    gpio_set_direction(dht_gpio_pin, GPIO_MODE_INPUT);
}

static void dht_gpio_set_output(void)
{
    gpio_set_direction(dht_gpio_pin, GPIO_MODE_OUTPUT);
}

static esp_err_t dht_read_data(uint8_t *data)
{
    uint8_t j;
    uint64_t low_time, high_time;

    dht_gpio_set_output();
    gpio_set_level(dht_gpio_pin, 0);
    esp_rom_delay_us(18000);
    gpio_set_level(dht_gpio_pin, 1);
    esp_rom_delay_us(40);
    dht_gpio_set_input();

    low_time = esp_timer_get_time();
    while (gpio_get_level(dht_gpio_pin) == 0) {
        if (esp_timer_get_time() - low_time > 100) return ESP_FAIL;
    }
    high_time = esp_timer_get_time();
    while (gpio_get_level(dht_gpio_pin) == 1) {
        if (esp_timer_get_time() - high_time > 100) return ESP_FAIL;
    }

    for (j = 0; j < DHT_DATA_BITS; j++) {
        low_time = esp_timer_get_time();
        while (gpio_get_level(dht_gpio_pin) == 0) {
            if (esp_timer_get_time() - low_time > 100) return ESP_FAIL;
        }
        high_time = esp_timer_get_time();
        while (gpio_get_level(dht_gpio_pin) == 1) {
            if (esp_timer_get_time() - high_time > 100) return ESP_FAIL;
        }

        data[j / 8] <<= 1;
        if ((esp_timer_get_time() - low_time) > 50) {
            data[j / 8] |= 1;
        }
    }

    return ESP_OK;
}

esp_err_t sensor_dht22_init(gpio_num_t dht_pin)
{
    dht_gpio_pin = dht_pin;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dht_gpio_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DHT22 GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "DHT22 sensor initialized on GPIO %d", dht_gpio_pin);
    return ESP_OK;
}

esp_err_t sensor_dht22_read(float *temperature, float *humidity)
{
    if (temperature == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[5] = {0, 0, 0, 0, 0};
    esp_err_t ret = dht_read_data(data);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw DHT22 data: %s", esp_err_to_name(ret));
        return ret;
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "DHT22 Checksum error!");
        return ESP_FAIL;
    }

    float current_humidity = (float)(((uint16_t)data[0] << 8) | data[1]) / 10.0f;
    float current_temperature = (float)(((uint16_t)data[2] << 8) | data[3]) / 10.0f;

    if (current_temperature > 125.0f || current_temperature < -40.0f ||
        current_humidity > 100.0f || current_humidity < 0.0f) {
        ESP_LOGW(TAG, "Invalid DHT22 readings: Temp=%.1f, Hum=%.1f", current_temperature, current_humidity);
        return ESP_FAIL;
    }

    temp_readings[ma_index] = current_temperature;
    hum_readings[ma_index] = current_humidity;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_temp = 0;
    float sum_hum = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_temp += temp_readings[i];
        sum_hum += hum_readings[i];
    }

    *temperature = sum_temp / count;
    *humidity = sum_hum / count;

    ESP_LOGD(TAG, "DHT22 Raw: Temp=%.1f, Hum=%.1f | Filtered: Temp=%.1f, Hum=%.1f",
             current_temperature, current_humidity, *temperature, *humidity);

    return ESP_OK;
}
