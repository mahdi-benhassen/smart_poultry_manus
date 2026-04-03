#include "sensor_water_level.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WATER_LEVEL_SENSOR";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t water_level_adc_channel;

#define MA_WINDOW_SIZE 10
static float water_level_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

#define WATER_LEVEL_MIN_VOLTAGE 1.8f
#define WATER_LEVEL_MAX_VOLTAGE 3.0f

esp_err_t sensor_water_level_init(adc_channel_t adc_channel)
{
    water_level_adc_channel = adc_channel;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, water_level_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Water level sensor initialized on ADC channel %d", water_level_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_water_level_read(float *water_level_pct)
{
    if (water_level_pct == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, water_level_adc_channel, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    int voltage = (adc_reading * 3300) / 4095;
    float sensor_voltage = (float)voltage / 1000.0f;

    float current_water_level_pct = 0.0f;
    if (sensor_voltage <= WATER_LEVEL_MIN_VOLTAGE) {
        current_water_level_pct = 100.0f;
    } else if (sensor_voltage >= WATER_LEVEL_MAX_VOLTAGE) {
        current_water_level_pct = 0.0f;
    } else {
        current_water_level_pct = (WATER_LEVEL_MAX_VOLTAGE - sensor_voltage) /
                                  (WATER_LEVEL_MAX_VOLTAGE - WATER_LEVEL_MIN_VOLTAGE) * 100.0f;
    }

    water_level_readings[ma_index] = current_water_level_pct;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_water_level = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_water_level += water_level_readings[i];
    }

    *water_level_pct = sum_water_level / count;

    ESP_LOGD(TAG, "Water Level Raw ADC: %d, Voltage: %.2fV, Raw Pct: %.2f%% | Filtered Pct: %.2f%%",
             adc_reading, sensor_voltage, current_water_level_pct, *water_level_pct);

    return ESP_OK;
}
