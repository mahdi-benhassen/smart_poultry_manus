#include "sensor_mq7.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MQ7_SENSOR";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t mq7_adc_channel;

#define MQ7_RL 10.0f
#define MQ7_RO 20.0f

#define MA_WINDOW_SIZE 10
static float co_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

static float get_co_ppm(float rs_ro_ratio)
{
    if (rs_ro_ratio > 0.6) return 10.0f;
    if (rs_ro_ratio > 0.4) return 30.0f;
    if (rs_ro_ratio > 0.2) return 100.0f;
    return 200.0f;
}

esp_err_t sensor_mq7_init(adc_channel_t adc_channel)
{
    mq7_adc_channel = adc_channel;

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
    ret = adc_oneshot_config_channel(adc_handle, mq7_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MQ7 sensor initialized on ADC channel %d", mq7_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_mq7_read(float *co_ppm)
{
    if (co_ppm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, mq7_adc_channel, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    int voltage = (adc_reading * 3300) / 4095;
    float sensor_voltage = (float)voltage / 1000.0f;
    float rs = (5.0f / sensor_voltage - 1.0f) * MQ7_RL;
    float rs_ro_ratio = rs / MQ7_RO;

    float current_co_ppm = get_co_ppm(rs_ro_ratio);

    co_readings[ma_index] = current_co_ppm;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_co = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_co += co_readings[i];
    }

    *co_ppm = sum_co / count;

    ESP_LOGD(TAG, "MQ7 Raw ADC: %d, Voltage: %.2fV, Rs/Ro: %.2f, Raw CO: %.2fppm | Filtered CO: %.2fppm",
             adc_reading, sensor_voltage, rs_ro_ratio, current_co_ppm, *co_ppm);

    return ESP_OK;
}
