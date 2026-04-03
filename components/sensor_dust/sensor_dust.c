#include "sensor_dust.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "DUST_SENSOR";
static adc_channel_t dust_adc_channel;
static gpio_num_t dust_led_pin;
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float dust_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

// GP2Y1010AU0F conversion parameters (typical values, may need calibration)
// V = 0.5V when dust density is 0ug/m3
// Sensitivity = 0.17mg/m3/V = 170ug/m3/V
#define DUST_VOLTAGE_OFFSET 0.5f // V
#define DUST_SENSITIVITY 170.0f  // ug/m3/V

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

esp_err_t sensor_dust_init(adc_channel_t adc_channel, gpio_num_t led_pin)
{
    dust_adc_channel = adc_channel;
    dust_led_pin = led_pin;

    // Configure LED pin as output
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
    gpio_set_level(dust_led_pin, 0); // Ensure LED is off initially

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_bytes = 1024,
        .conv_frame_size = 100,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_channel_config_t chan_config = {
        .channel = dust_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_continuous_config_channel(adc_handle, &chan_config));

    if (!adc_calibration_init(ADC_UNIT_1, dust_adc_channel, &adc_cali_handle)) {
        ESP_LOGE(TAG, "ADC calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Dust sensor initialized (ADC channel %d, LED pin %d)", dust_adc_channel, dust_led_pin);
    return ESP_OK;
}

esp_err_t sensor_dust_read(float *dust_ugm3)
{
    if (dust_ugm3 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t adc_reading = 0;
    uint8_t result[100];
    uint32_t ret_num = 0;

    // Turn on LED
    gpio_set_level(dust_led_pin, 1);
    ets_delay_us(280); // Wait for 0.28ms (typical pulse width)

    esp_err_t ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC continuous conversion: %s", esp_err_to_name(ret));
        gpio_set_level(dust_led_pin, 0); // Turn off LED
        return ret;
    }

    ret = adc_continuous_read(adc_handle, result, 100, &ret_num, 100); // Read 100 bytes, timeout 100ms
    if (ret == ESP_OK && ret_num > 0) {
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_result_t *p = (adc_digi_result_t*)&result[i];
            if (p->channel == dust_adc_channel) {
                adc_reading = p->data;
                break;
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to read ADC data: %s", esp_err_to_name(ret));
        adc_continuous_stop(adc_handle);
        gpio_set_level(dust_led_pin, 0); // Turn off LED
        return ret;
    }
    adc_continuous_stop(adc_handle);

    // Turn off LED
    gpio_set_level(dust_led_pin, 0);

    int voltage = 0;
    if (adc_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage));
    } else {
        ESP_LOGW(TAG, "ADC calibration handle not available, using raw value");
        voltage = (adc_reading * 3300) / 4095; // Assuming 3.3V reference and 12-bit ADC
    }

    float sensor_voltage = (float)voltage / 1000.0f; // Convert mV to V

    // Convert voltage to dust density
    float current_dust_ugm3 = (sensor_voltage - DUST_VOLTAGE_OFFSET) * DUST_SENSITIVITY;
    if (current_dust_ugm3 < 0) current_dust_ugm3 = 0; // Dust concentration cannot be negative

    // Apply moving average filter
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
