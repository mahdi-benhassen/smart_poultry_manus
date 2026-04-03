#include "sensor_sound.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "SOUND_SENSOR";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t sound_adc_channel;

#define MA_WINDOW_SIZE 10
static float sound_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

#define SOUND_SAMPLE_COUNT 256
#define SOUND_REFERENCE_VOLTAGE 3.3f
#define SOUND_MAX_ADC_VALUE 4095.0f

static float voltage_to_db(float rms_voltage)
{
    if (rms_voltage <= 0) return 0.0f;
    return 20.0f * log10f(rms_voltage / 0.001f);
}

esp_err_t sensor_sound_init(adc_channel_t adc_channel)
{
    sound_adc_channel = adc_channel;

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
    ret = adc_oneshot_config_channel(adc_handle, sound_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Sound sensor initialized on ADC channel %d", sound_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_sound_read(float *sound_db)
{
    if (sound_db == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float sum_squares = 0;
    int valid_samples = 0;

    for (int i = 0; i < SOUND_SAMPLE_COUNT; i++) {
        int adc_reading = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, sound_adc_channel, &adc_reading);
        if (ret == ESP_OK) {
            float voltage = (float)(adc_reading * (int)(SOUND_REFERENCE_VOLTAGE * 1000)) / (int)SOUND_MAX_ADC_VALUE / 1000.0f;
            sum_squares += voltage * voltage;
            valid_samples++;
        }
    }

    if (valid_samples > 0) {
        float rms_voltage = sqrtf(sum_squares / valid_samples);
        float current_sound_db = voltage_to_db(rms_voltage);

        sound_readings[ma_index] = current_sound_db;
        ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

        if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
            ma_initialized = true;
        }

        float sum_db = 0;
        int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

        for (int i = 0; i < count; i++) {
            sum_db += sound_readings[i];
        }

        *sound_db = sum_db / count;

        ESP_LOGD(TAG, "Sound Raw RMS Voltage: %.4fV, Raw dB: %.2fdB | Filtered dB: %.2fdB",
                 rms_voltage, current_sound_db, *sound_db);
    } else {
        ESP_LOGE(TAG, "No valid samples read for sound sensor");
        return ESP_FAIL;
    }

    return ESP_OK;
}
