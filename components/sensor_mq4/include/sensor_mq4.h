#ifndef SENSOR_MQ4_H
#define SENSOR_MQ4_H

#include "esp_err.h"
#include "driver/adc.h"

/**
 * @brief Initializes the MQ4 sensor.
 * 
 * @param adc_channel The ADC channel connected to the MQ4 analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq4_init(adc_channel_t adc_channel);

/**
 * @brief Reads Methane concentration from the MQ4 sensor.
 * 
 * @param methane_ppm Pointer to a float to store the Methane concentration in ppm.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq4_read(float *methane_ppm);

#endif // SENSOR_MQ4_H
