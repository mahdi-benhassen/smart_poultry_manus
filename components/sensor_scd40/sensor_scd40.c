#include "sensor_scd40.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

static const char *TAG = "SCD40_SENSOR";

#define SCD40_SENSOR_ADDR 0x62
#define SCD40_CMD_START_PERIODIC_MEASUREMENT 0x21ac
#define SCD40_CMD_READ_MEASUREMENT 0x0368
#define SCD40_CMD_STOP_PERIODIC_MEASUREMENT 0x3f86

// Moving average filter parameters
#define MA_WINDOW_SIZE 10
static float co2_readings[MA_WINDOW_SIZE];
static float temp_readings[MA_WINDOW_SIZE];
static float hum_readings[MA_WINDOW_SIZE];
static int ma_index = 0;
static bool ma_initialized = false;

static i2c_port_t i2c_master_port;

static esp_err_t scd40_write_command(uint16_t command)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (command >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, command & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command 0x%04x: %s", command, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t scd40_read_data(uint16_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    for (size_t i = 0; i < len; i++) {
        uint8_t msb, lsb;
        i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &lsb, I2C_MASTER_ACK);
        uint8_t crc;
        i2c_master_read_byte(cmd, &crc, (i == len - 1) ? I2C_MASTER_NACK : I2C_MASTER_ACK);
        data[i] = (msb << 8) | lsb;
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sensor_scd40_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
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

    // Stop any ongoing measurements
    ESP_ERROR_CHECK(scd40_write_command(SCD40_CMD_STOP_PERIODIC_MEASUREMENT));
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for sensor to stop

    // Start periodic measurement
    ESP_ERROR_CHECK(scd40_write_command(SCD40_CMD_START_PERIODIC_MEASUREMENT));
    ESP_LOGI(TAG, "SCD40 sensor initialized on I2C port %d (SDA:%d, SCL:%d)", i2c_master_port, sda_pin, scl_pin);
    return ESP_OK;
}

esp_err_t sensor_scd40_read(float *co2_ppm, float *temperature, float *humidity)
{
    if (co2_ppm == NULL || temperature == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_data[3]; // CO2, Temperature, Humidity
    esp_err_t ret = scd40_read_data(raw_data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD40 measurement");
        return ret;
    }

    // Convert raw data to actual values
    float current_co2 = (float)raw_data[0];
    float current_temperature = -45.0f + 175.0f * ((float)raw_data[1] / 65536.0f);
    float current_humidity = 100.0f * ((float)raw_data[2] / 65536.0f);

    // Apply moving average filter
    co2_readings[ma_index] = current_co2;
    temp_readings[ma_index] = current_temperature;
    hum_readings[ma_index] = current_humidity;
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    if (!ma_initialized && ma_index == MA_WINDOW_SIZE - 1) {
        ma_initialized = true;
    }

    float sum_co2 = 0;
    float sum_temp = 0;
    float sum_hum = 0;
    int count = ma_initialized ? MA_WINDOW_SIZE : ma_index;

    for (int i = 0; i < count; i++) {
        sum_co2 += co2_readings[i];
        sum_temp += temp_readings[i];
        sum_hum += hum_readings[i];
    }

    *co2_ppm = sum_co2 / count;
    *temperature = sum_temp / count;
    *humidity = sum_hum / count;

    ESP_LOGD(TAG, "SCD40 Raw: CO2=%.0fppm, Temp=%.2f, Hum=%.2f | Filtered: CO2=%.0fppm, Temp=%.2f, Hum=%.2f",
             current_co2, current_temperature, current_humidity, *co2_ppm, *temperature, *humidity);

    return ESP_OK;
}
