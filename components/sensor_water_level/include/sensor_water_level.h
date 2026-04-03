#ifndef SENSOR_WATER_LEVEL_H
#define SENSOR_WATER_LEVEL_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

/**
 * @brief Initializes the water level sensor.
 *
 * @param adc_channel The ADC channel connected to the water level sensor analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_water_level_init(adc_channel_t adc_channel);

/**
 * @brief Reads water level from the sensor and returns it as a percentage.
 *
 * @param water_level_pct Pointer to a float to store the water level in percentage (0-100).
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_water_level_read(float *water_level_pct);

#endif // SENSOR_WATER_LEVEL_H
