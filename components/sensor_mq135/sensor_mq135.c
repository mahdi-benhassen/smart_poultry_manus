#include "sensor_mq135.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MQ135_SENSOR";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t mq135_adc_channel;

#define MQ135_RL 10.0f
#define MQ135_RO 76.63f

static float get_ppm_from_rs_ro(float rs_ro_ratio)
{
    if (rs_ro_ratio > 1.0f) return 10.0f;
    if (rs_ro_ratio > 0.8f) return 20.0f;
    if (rs_ro_ratio > 0.6f) return 50.0f;
    if (rs_ro_ratio > 0.4f) return 100.0f;
    return 200.0f;
}

#define MA_WINDOW_SIZE 10
static float nh3_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

esp_err_t sensor_mq135_init(adc_channel_t adc_channel)
{
    mq135_adc_channel = adc_channel;

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
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, mq135_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MQ135 sensor initialized on ADC channel %d", mq135_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_mq135_read(float *nh3_ppm)
{
    if (nh3_ppm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, mq135_adc_channel, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    int voltage = (adc_reading * 3300) / 4095;
    float sensor_voltage = (float)voltage / 1000.0f;
    float rs = (5.0f / sensor_voltage - 1.0f) * MQ135_RL;
    float rs_ro_ratio = rs / MQ135_RO;

    float current_nh3_ppm = get_ppm_from_rs_ro(rs_ro_ratio);

    nh3_readings[ma_index] = current_nh3_ppm;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_nh3 = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_nh3 += nh3_readings[i];
    }

    *nh3_ppm = sum_nh3 / count;

    ESP_LOGD(TAG, "MQ135 Raw ADC: %d, Voltage: %.2fV, Rs/Ro: %.2f, Raw NH3: %.2fppm | Filtered NH3: %.2fppm",
             adc_reading, sensor_voltage, rs_ro_ratio, current_nh3_ppm, *nh3_ppm);

    return ESP_OK;
}
