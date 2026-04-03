#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Structure to hold all sensor readings
typedef struct {
    float temperature;
    float humidity;
    float nh3_ppm;
    float co_ppm;
    float co2_ppm;
    float methane_ppm;
    float h2s_ppm;
    float dust_ugm3;
    float light_lux;
    float water_level_pct;
    float sound_db;
    bool door_open;
} sensor_data_t;

/**
 * @brief Initializes the sensor manager and all registered sensors.
 * 
 * This function should be called once at system startup.
 * 
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_manager_init(void);

/**
 * @brief Reads data from all registered sensors.
 * 
 * This function iterates through all initialized sensors, calls their respective
 * read functions, and updates the internal sensor data structure.
 * 
 * @return ESP_OK on success, or an error code if any sensor read fails.
 */
esp_err_t sensor_manager_read_all(void);

/**
 * @brief Retrieves the latest sensor data.
 * 
 * This function provides a thread-safe way to get the current sensor readings.
 * 
 * @param[out] data Pointer to a sensor_data_t structure to fill with the latest data.
 * @return ESP_OK on success, or an error code if the data cannot be retrieved.
 */
esp_err_t sensor_manager_get_data(sensor_data_t *data);

#endif // SENSOR_MANAGER_H
