#include "sensor_sound.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "SOUND_SENSOR";
static adc_channel_t sound_adc_channel;
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float sound_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

// Sound sensor parameters (example, needs actual calibration)
#define SOUND_SAMPLE_COUNT 256 // Number of samples to take for RMS calculation
#define SOUND_REFERENCE_VOLTAGE 3.3f // ADC reference voltage in V
#define SOUND_MAX_ADC_VALUE 4095.0f // 12-bit ADC

// Conversion from voltage to dB (example, needs calibration)
// Assuming 0dB is at 1V RMS, and sensitivity is 20dB/decade
static float voltage_to_db(float rms_voltage)
{
    if (rms_voltage <= 0) return 0.0f; // Avoid log(0)
    // This is a very simplified model. Real sound sensors often have specific calibration curves.
    return 20.0f * log10f(rms_voltage / 0.001f); // Reference 1mV for 0dB
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_cali_handle_t *out_handle)
{
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, out_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    if (!calibrated) {
        ESP_LOGE(TAG, "No supported ADC calibration scheme found");
    }

    return calibrated;
}

esp_err_t sensor_sound_init(adc_channel_t adc_channel)
{
    sound_adc_channel = adc_channel;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_bytes = 1024,
        .conv_frame_size = SOUND_SAMPLE_COUNT, // Read multiple samples for RMS
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_channel_config_t chan_config = {
        .channel = sound_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_continuous_config_channel(adc_handle, &chan_config));

    if (!adc_calibration_init(ADC_UNIT_1, sound_adc_channel, &adc_cali_handle)) {
        ESP_LOGE(TAG, "ADC calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sound sensor initialized on ADC channel %d", sound_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_sound_read(float *sound_db)
{
    if (sound_db == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t adc_reading_sum = 0;
    uint8_t result[SOUND_SAMPLE_COUNT * SOC_ADC_DIGI_RESULT_BYTES];
    uint32_t ret_num = 0;

    esp_err_t ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC continuous conversion: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read multiple samples for RMS calculation
    ret = adc_continuous_read(adc_handle, result, sizeof(result), &ret_num, 100); 
    if (ret == ESP_OK && ret_num > 0) {
        float sum_squares = 0;
        int valid_samples = 0;
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_result_t *p = (adc_digi_result_t*)&result[i];
            if (p->channel == sound_adc_channel) {
                int voltage_mv = 0;
                if (adc_cali_handle) {
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, p->data, &voltage_mv));
                } else {
                    voltage_mv = (p->data * (int)(SOUND_REFERENCE_VOLTAGE * 1000)) / (int)SOUND_MAX_ADC_VALUE;
                }
                float voltage = (float)voltage_mv / 1000.0f;
                sum_squares += voltage * voltage;
                valid_samples++;
            }
        }
        if (valid_samples > 0) {
            float rms_voltage = sqrtf(sum_squares / valid_samples);
            float current_sound_db = voltage_to_db(rms_voltage);

            // Apply moving average filter
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
            ret = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Failed to read ADC data: %s", esp_err_to_name(ret));
    }
    adc_continuous_stop(adc_handle);

    return ret;
}
