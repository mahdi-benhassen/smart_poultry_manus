#include "sensor_water_level.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WATER_LEVEL_SENSOR";
static adc_channel_t water_level_adc_channel;
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float water_level_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

// Water level sensor calibration values (example, needs actual calibration)
// Assuming a sensor where higher voltage means lower water level
#define WATER_LEVEL_MIN_VOLTAGE 1.8f // Voltage when water level is 100% (sensor fully submerged)
#define WATER_LEVEL_MAX_VOLTAGE 3.0f // Voltage when water level is 0% (sensor dry)

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

esp_err_t sensor_water_level_init(adc_channel_t adc_channel)
{
    water_level_adc_channel = adc_channel;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_bytes = 1024,
        .conv_frame_size = 100,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_channel_config_t chan_config = {
        .channel = water_level_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_continuous_config_channel(adc_handle, &chan_config));

    if (!adc_calibration_init(ADC_UNIT_1, water_level_adc_channel, &adc_cali_handle)) {
        ESP_LOGE(TAG, "ADC calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Water level sensor initialized on ADC channel %d", water_level_adc_channel);
    return ESP_OK;
}

esp_err_t sensor_water_level_read(float *water_level_pct)
{
    if (water_level_pct == NULL) {
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
            if (p->channel == water_level_adc_channel) {
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

    int voltage_mv = 0;
    if (adc_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv));
    } else {
        ESP_LOGW(TAG, "ADC calibration handle not available, using raw value");
        voltage_mv = (adc_reading * 3300) / 4095; // Assuming 3.3V reference and 12-bit ADC
    }

    float sensor_voltage = (float)voltage_mv / 1000.0f; // Convert mV to V

    // Map voltage to percentage
    float current_water_level_pct = 0.0f;
    if (sensor_voltage <= WATER_LEVEL_MIN_VOLTAGE) {
        current_water_level_pct = 100.0f; // Fully submerged
    } else if (sensor_voltage >= WATER_LEVEL_MAX_VOLTAGE) {
        current_water_level_pct = 0.0f;   // Dry
    } else {
        // Linear interpolation: (V - V_max) / (V_min - V_max) * 100
        current_water_level_pct = (WATER_LEVEL_MAX_VOLTAGE - sensor_voltage) / 
                                  (WATER_LEVEL_MAX_VOLTAGE - WATER_LEVEL_MIN_VOLTAGE) * 100.0f;
    }

    // Apply moving average filter
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
