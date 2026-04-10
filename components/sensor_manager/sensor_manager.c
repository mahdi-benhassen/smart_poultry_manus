#include "sensor_manager.h"
#include "esp_log.h"
#include "freertos/task.h"

// Include headers for individual sensors
#include "sensor_dht22.h"
#include "sensor_mq135.h"
#include "sensor_mq7.h"
#include "sensor_mq4.h"
#include "sensor_mq136.h"
#include "sensor_scd40.h"
#include "sensor_dust.h"
#include "sensor_light.h"
#include "sensor_water_level.h"
#include "sensor_sound.h"
#include "sensor_door.h"

// Kconfig values
#include "sdkconfig.h"

static const char *TAG = "SENSOR_MANAGER";

static sensor_data_t s_sensor_data;
static SemaphoreHandle_t s_sensor_data_mutex;

esp_err_t sensor_manager_init(void)
{
    s_sensor_data_mutex = xSemaphoreCreateMutex();
    if (s_sensor_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor data mutex");
        return ESP_FAIL;
    }

    // Initialize individual sensors
    ESP_ERROR_CHECK(sensor_dht22_init(CONFIG_SPS_DHT22_PIN));
    ESP_ERROR_CHECK(sensor_mq135_init(CONFIG_SPS_MQ135_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_mq7_init(CONFIG_SPS_MQ7_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_mq4_init(CONFIG_SPS_MQ4_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_mq136_init(CONFIG_SPS_MQ136_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_scd40_init(I2C_NUM_0, CONFIG_SPS_I2C_SDA_PIN, CONFIG_SPS_I2C_SCL_PIN));
    ESP_ERROR_CHECK(sensor_dust_init(CONFIG_SPS_DUST_ADC_CHANNEL, CONFIG_SPS_DUST_LED_PIN));
    ESP_ERROR_CHECK(sensor_light_init(I2C_NUM_0, CONFIG_SPS_I2C_SDA_PIN, CONFIG_SPS_I2C_SCL_PIN));
    ESP_ERROR_CHECK(sensor_water_level_init(CONFIG_SPS_WATER_LEVEL_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_sound_init(CONFIG_SPS_SOUND_ADC_CHANNEL));
    ESP_ERROR_CHECK(sensor_door_init(CONFIG_SPS_DOOR_PIN));

    return ESP_OK;
}

esp_err_t sensor_manager_read_all(void)
{
    if (xSemaphoreTake(s_sensor_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        esp_err_t status = ESP_OK;
        esp_err_t err;

        // Read from individual sensors and update s_sensor_data only on success
        float temp, hum;
        err = sensor_dht22_read(&temp, &hum);
        if (err == ESP_OK) {
            s_sensor_data.temperature = temp;
            s_sensor_data.humidity = hum;
        } else {
            ESP_LOGW(TAG, "DHT22 read failed: %s", esp_err_to_name(err));
            status = ESP_FAIL;
        }

        float nh3;
        err = sensor_mq135_read(&nh3);
        if (err == ESP_OK) s_sensor_data.nh3_ppm = nh3;
        else { ESP_LOGW(TAG, "MQ135 read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float co;
        err = sensor_mq7_read(&co);
        if (err == ESP_OK) s_sensor_data.co_ppm = co;
        else { ESP_LOGW(TAG, "MQ7 read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float ch4;
        err = sensor_mq4_read(&ch4);
        if (err == ESP_OK) s_sensor_data.methane_ppm = ch4;
        else { ESP_LOGW(TAG, "MQ4 read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float h2s;
        err = sensor_mq136_read(&h2s);
        if (err == ESP_OK) s_sensor_data.h2s_ppm = h2s;
        else { ESP_LOGW(TAG, "MQ136 read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float scd40_co2, scd40_temp, scd40_hum;
        err = sensor_scd40_read(&scd40_co2, &scd40_temp, &scd40_hum);
        if (err == ESP_OK) {
            s_sensor_data.co2_ppm = scd40_co2;
        } else {
            ESP_LOGW(TAG, "SCD40 read failed: %s", esp_err_to_name(err));
            status = ESP_FAIL;
        }

        float dust;
        err = sensor_dust_read(&dust);
        if (err == ESP_OK) s_sensor_data.dust_ugm3 = dust;
        else { ESP_LOGW(TAG, "Dust read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float lux;
        err = sensor_light_read(&lux);
        if (err == ESP_OK) s_sensor_data.light_lux = lux;
        else { ESP_LOGW(TAG, "Light read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float water;
        err = sensor_water_level_read(&water);
        if (err == ESP_OK) s_sensor_data.water_level_pct = water;
        else { ESP_LOGW(TAG, "Water level read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        float sound;
        err = sensor_sound_read(&sound);
        if (err == ESP_OK) s_sensor_data.sound_db = sound;
        else { ESP_LOGW(TAG, "Sound read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        bool door_open;
        err = sensor_door_read(&door_open);
        if (err == ESP_OK) s_sensor_data.door_open = door_open;
        else { ESP_LOGW(TAG, "Door sensor read failed: %s", esp_err_to_name(err)); status = ESP_FAIL; }

        xSemaphoreGive(s_sensor_data_mutex);
        return status;
    }
    ESP_LOGE(TAG, "Failed to take sensor data mutex");
    return ESP_FAIL;
}

esp_err_t sensor_manager_get_data(sensor_data_t *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_sensor_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        *data = s_sensor_data;
        xSemaphoreGive(s_sensor_data_mutex);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to take sensor data mutex");
    return ESP_FAIL;
}
