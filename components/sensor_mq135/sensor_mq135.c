#include "sensor_mq135.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MQ135_SENSOR";
static adc_channel_t mq135_adc_channel;
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// MQ135 specific parameters (these might need calibration for accuracy)
#define MQ135_RL 10.0f // Load resistance in kOhm
#define MQ135_RO 76.63f // Ro in clean air (kOhm), needs calibration

// Simplified conversion curve (example, needs actual calibration data)
// This is a placeholder, actual values depend on sensor and environment
static float get_ppm_from_rs_ro(float rs_ro_ratio)
{
    // Example curve for NH3 (MQ135 also detects other gases)
    // This is a very rough approximation. Real application needs proper calibration.
    if (rs_ro_ratio > 1.0f) return 10.0f; // Placeholder
    if (rs_ro_ratio > 0.8f) return 20.0f;
    if (rs_ro_ratio > 0.6f) return 50.0f;
    if (rs_ro_ratio > 0.4f) return 100.0f;
    return 200.0f; // Placeholder
}

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float nh3_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

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

esp_err_t sensor_mq135_init(adc_channel_t adc_channel)
{
    mq135_adc_channel = adc_channel;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_bytes = 1024,
        .conv_frame_size = 100,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_channel_config_t chan_config = {
        .channel = mq135_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_continuous_config_channel(adc_handle, &chan_config));

    if (!adc_calibration_init(ADC_UNIT_1, mq135_adc_channel, &adc_cali_handle)) {
        ESP_LOGE(TAG, "ADC calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQ135 sensor initialized on ADC channel %d", mq135_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_mq135_read(float *nh3_ppm)
{
    if (nh3_ppm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t adc_reading = 0;
    uint8_t result[100];
    uint32_t ret_num = 0;

    esp_err_t ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC continuous conversion: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_continuous_read(adc_handle, result, 100, &ret_num, 100); // Read 100 bytes, timeout 100ms
    if (ret == ESP_OK && ret_num > 0) {
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_result_t *p = (adc_digi_result_t*)&result[i];
            if (p->channel == mq135_adc_channel) {
                adc_reading = p->data;
                break;
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to read ADC data: %s", esp_err_to_name(ret));
        adc_continuous_stop(adc_handle);
        return ret;
    }
    adc_continuous_stop(adc_handle);

    int voltage = 0;
    if (adc_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage));
    } else {
        ESP_LOGW(TAG, "ADC calibration handle not available, using raw value");
        // Fallback if calibration fails, very rough approximation
        voltage = (adc_reading * 3300) / 4095; // Assuming 3.3V reference and 12-bit ADC
    }

    float sensor_voltage = (float)voltage / 1000.0f; // Convert mV to V
    float rs = (5.0f / sensor_voltage - 1.0f) * MQ135_RL; // Assuming 5V VCC for sensor
    float rs_ro_ratio = rs / MQ135_RO;

    float current_nh3_ppm = get_ppm_from_rs_ro(rs_ro_ratio);

    // Apply moving average filter
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
