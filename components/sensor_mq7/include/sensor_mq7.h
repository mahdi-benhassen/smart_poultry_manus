#ifndef SENSOR_MQ7_H
#define SENSOR_MQ7_H

#include "esp_err.h"
#include "driver/adc.h"

/**
 * @brief Initializes the MQ7 sensor.
 * 
 * @param adc_channel The ADC channel connected to the MQ7 analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq7_init(adc_channel_t adc_channel);

/**
 * @brief Reads CO concentration from the MQ7 sensor.
 * 
 * @param co_ppm Pointer to a float to store the CO concentration in ppm.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq7_read(float *co_ppm);

#endif // SENSOR_MQ7_H
