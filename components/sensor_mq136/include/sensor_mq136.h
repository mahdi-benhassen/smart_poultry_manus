#ifndef SENSOR_MQ136_H
#define SENSOR_MQ136_H

#include "esp_err.h"
#include "driver/adc.h"

/**
 * @brief Initializes the MQ136 sensor.
 * 
 * @param adc_channel The ADC channel connected to the MQ136 analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq136_init(adc_channel_t adc_channel);

/**
 * @brief Reads H2S concentration from the MQ136 sensor.
 * 
 * @param h2s_ppm Pointer to a float to store the H2S concentration in ppm.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_mq136_read(float *h2s_ppm);

#endif // SENSOR_MQ136_H
