#include "sensor_dust.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "DUST_SENSOR";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t dust_adc_channel;
static gpio_num_t dust_led_pin;

#define MA_WINDOW_SIZE 10
static float dust_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

#define DUST_VOLTAGE_OFFSET 0.5f
#define DUST_SENSITIVITY 170.0f

esp_err_t sensor_dust_init(adc_channel_t adc_channel, gpio_num_t led_pin)
{
    dust_adc_channel = adc_channel;
    dust_led_pin = led_pin;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dust_led_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Dust LED GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(dust_led_pin, 0);

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, dust_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Dust sensor initialized (ADC channel %d, LED pin %d)", dust_adc_channel, dust_led_pin);
    return ESP_OK;
}

esp_err_t sensor_dust_read(float *dust_ugm3)
{
    if (dust_ugm3 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc_reading = 0;

    gpio_set_level(dust_led_pin, 1);
    ets_delay_us(280);

    esp_err_t ret = adc_oneshot_read(adc_handle, dust_adc_channel, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        gpio_set_level(dust_led_pin, 0);
        return ret;
    }

    gpio_set_level(dust_led_pin, 0);

    int voltage = (adc_reading * 3300) / 4095;
    float sensor_voltage = (float)voltage / 1000.0f;

    float current_dust_ugm3 = (sensor_voltage - DUST_VOLTAGE_OFFSET) * DUST_SENSITIVITY;
    if (current_dust_ugm3 < 0) current_dust_ugm3 = 0;

    dust_readings[ma_index] = current_dust_ugm3;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_dust = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_dust += dust_readings[i];
    }

    *dust_ugm3 = sum_dust / count;

    ESP_LOGD(TAG, "Dust Raw ADC: %d, Voltage: %.2fV, Raw Dust: %.2fug/m3 | Filtered Dust: %.2fug/m3",
             adc_reading, sensor_voltage, current_dust_ugm3, *dust_ugm3);

    return ESP_OK;
}
