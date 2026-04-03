#ifndef SENSOR_MQ135_H
#define SENSOR_MQ135_H

#include "esp_err.h"
#include "driver/adc.h"

/**
 * @brief Initializes the MQ135 sensor.
 * 
 * @param adc_channel The ADC channel connected to the MQ135 analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq135_init(adc_channel_t adc_channel);

/**
 * @brief Reads NH3 concentration from the MQ135 sensor.
 * 
 * @param nh3_ppm Pointer to a float to store the NH3 concentration in ppm.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq135_read(float *nh3_ppm);

#endif // SENSOR_MQ135_H
