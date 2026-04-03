#include "sensor_light.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

static const char *TAG = "BH1750_SENSOR";

#define BH1750_SENSOR_ADDR 0x23 // BH1750 I2C address
#define BH1750_POWER_ON 0x01    // Power On
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10 // Continuous H-Res Mode

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float lux_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

static i2c_port_t i2c_master_port;

esp_err_t sensor_light_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    i2c_master_port = i2c_port;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz
    };
    esp_err_t ret = i2c_param_config(i2c_master_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Power on and set continuous high resolution mode
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_POWER_ON, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on BH1750: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for power-on

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_CONTINUOUS_HIGH_RES_MODE, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BH1750 mode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(180)); // Measurement time for high-res mode

    ESP_LOGI(TAG, "BH1750 light sensor initialized on I2C port %d (SDA:%d, SCL:%d)", i2c_master_port, sda_pin, scl_pin);
    return ESP_OK;
}

esp_err_t sensor_light_read(float *light_lux)
{
    if (light_lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BH1750 data: %s", esp_err_to_name(ret));
        return ret;
    }

    uint16_t raw_lux = (data[0] << 8) | data[1];
    float current_lux = (float)raw_lux / 1.2f; // Convert raw value to lux

    // Apply moving average filter
    lux_readings[ma_index] = current_lux;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_lux = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_lux += lux_readings[i];
    }

    *light_lux = sum_lux / count;

    ESP_LOGD(TAG, "BH1750 Raw Lux: %d, Raw Lux: %.2f | Filtered Lux: %.2f",
             raw_lux, current_lux, *light_lux);

    return ESP_OK;
}
