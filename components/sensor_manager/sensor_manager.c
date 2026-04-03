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
        // Read from individual sensors and update s_sensor_data
        sensor_dht22_read(&s_sensor_data.temperature, &s_sensor_data.humidity);
        sensor_mq135_read(&s_sensor_data.nh3_ppm);
        sensor_mq7_read(&s_sensor_data.co_ppm);
        sensor_mq4_read(&s_sensor_data.methane_ppm);
        sensor_mq136_read(&s_sensor_data.h2s_ppm);

        float scd40_co2, scd40_temp, scd40_hum;
        sensor_scd40_read(&scd40_co2, &scd40_temp, &scd40_hum);
        s_sensor_data.co2_ppm = scd40_co2;
        // Optionally use SCD40 temp/hum if DHT22 fails or for cross-validation

        sensor_dust_read(&s_sensor_data.dust_ugm3);
        sensor_light_read(&s_sensor_data.light_lux);
        sensor_water_level_read(&s_sensor_data.water_level_pct);
        sensor_sound_read(&s_sensor_data.sound_db);
        sensor_door_read(&s_sensor_data.door_open);

        xSemaphoreGive(s_sensor_data_mutex);
        return ESP_OK;
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
