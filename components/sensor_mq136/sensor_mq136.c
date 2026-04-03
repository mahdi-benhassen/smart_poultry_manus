#include "sensor_mq136.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MQ136_SENSOR";
static adc_channel_t mq136_adc_channel;
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// MQ136 specific parameters (these might need calibration for accuracy)
#define MQ136_RL 10.0f // Load resistance in kOhm
#define MQ136_RO 10.0f // Ro in clean air (kOhm), needs calibration

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float h2s_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

// Placeholder for H2S conversion curve (Rs/Ro to ppm)
// This is a generic example. Actual values should come from sensor datasheet or calibration.
static float get_h2s_ppm(float rs_ro_ratio)
{
    // Example curve based on typical MQ-136 datasheet characteristics
    // For H2S, higher Rs/Ro often means higher concentration
    // This is a simplified model, real calibration is crucial.
    if (rs_ro_ratio < 0.5) return 1.0f; // Low H2S
    if (rs_ro_ratio < 1.0) return 5.0f;
    if (rs_ro_ratio < 2.0) return 10.0f;
    return 20.0f; // High H2S
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

esp_err_t sensor_mq136_init(adc_channel_t adc_channel)
{
    mq136_adc_channel = adc_channel;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_bytes = 1024,
        .conv_frame_size = 100,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_channel_config_t chan_config = {
        .channel = mq136_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_continuous_config_channel(adc_handle, &chan_config));

    if (!adc_calibration_init(ADC_UNIT_1, mq136_adc_channel, &adc_cali_handle)) {
        ESP_LOGE(TAG, "ADC calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQ136 sensor initialized on ADC channel %d", mq136_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_mq136_read(float *h2s_ppm)
{
    if (h2s_ppm == NULL) {
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
            if (p->channel == mq136_adc_channel) {
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
        voltage = (adc_reading * 3300) / 4095; // Assuming 3.3V reference and 12-bit ADC
    }

    float sensor_voltage = (float)voltage / 1000.0f; // Convert mV to V
    // Assuming VCC = 5V for MQ sensors, voltage divider with RL
    // Rs = (VCC / V_sensor - 1) * RL
    float rs = (5.0f / sensor_voltage - 1.0f) * MQ136_RL;
    float rs_ro_ratio = rs / MQ136_RO;

    float current_h2s_ppm = get_h2s_ppm(rs_ro_ratio);

    // Apply moving average filter
    h2s_readings[ma_index] = current_h2s_ppm;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_h2s = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_h2s += h2s_readings[i];
    }

    *h2s_ppm = sum_h2s / count;

    ESP_LOGD(TAG, "MQ136 Raw ADC: %d, Voltage: %.2fV, Rs/Ro: %.2f, Raw H2S: %.2fppm | Filtered H2S: %.2fppm",
             adc_reading, sensor_voltage, rs_ro_ratio, current_h2s_ppm, *h2s_ppm);

    return ESP_OK;
}
